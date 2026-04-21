# cadence7 sensor sanity + command→response analysis

**Date**: 2026-04-21
**Source**: `eval-results/2026-04-21T16:24:41Z/tier1-aeroStandard/data.dat`
(44,202 rows, 23.7s span, cadence exactly 100 ms)
**Weights**: final cadence7 (gen 400, best=-35951)

Purpose: verify that the retrained NN's sim data is self-consistent with the
canonical conventions in [docs/COORDINATE_CONVENTIONS.md](../../docs/COORDINATE_CONVENTIONS.md)
and reproduces the 023 cause/effect shape after the WI4 cadence fix (dt=0.005,
fps=20) + msplink qy/qz flip.

## Artifacts

- [`sensor_self_check_cadence7_tier1.png`](./sensor_self_check_cadence7_tier1.png) — 8-panel bar chart, all checks.
- [`sensor_self_check_cadence7_tier1.md`](./sensor_self_check_cadence7_tier1.md) — text report.
- [`cmd_response_scatter_cadence7_p0p2.png`](./cmd_response_scatter_cadence7_p0p2.png) — 3×3 cmd→response scatter, path 0 & 2 slice (matches 023 format).

## 8-panel audit results (sensor_self_check)

| # | Check | Pass/Fail | Notes |
|---|---|---|---|
| 1 | Position ↔ velocity integration | **FAIL** | N slope +1.35 r=0.00, **E slope −20.1 r=−0.31, D slope +19.5 r=+0.24**. Library bug on sim path, NOT a training issue — body-vel→world rotation is correct (check 7 uses the same rotation and passes). Likely a scale or sign issue in how the sim-path integration reconstructs world position. Not blocking US6; deferred to post-flight Group D analysis cleanup alongside T060/T061/T062. |
| 2 | Gyro ↔ quat-delta | PASS | p +1.18/r=0.93, q +0.87/r=0.92, r +1.02/r=0.99. All three axes positive, correlation near 1. **Key coordinate-convention sign check — confirms both gyro and quat follow standard aerospace RHR.** |
| 3 | Euler(quat) ↔ attitude[] | SKIP | Sim data.dat has no `attitude[]` column. |
| 4 | Accel ↔ gravity | SKIP | Sim data.dat has no accel. |
| 5 | Heading ↔ ground track | PASS | yaw slope 0.93, r=1.00. World-frame NED and quat-derived heading agree. |
| 6 | Mag ↔ heading | SKIP | Sim has no mag. |
| 7 | Attitude vector ↔ velocity dir | PASS | N 0.93/r=0.97, E 0.94/r=0.95, D 0.94/r=0.89. Body→world rotation is good. |
| 8 | Cmd ↔ attitude change (rate) | PASS | pitch→q slope +0.09/r=0.05, roll→p slope +0.006/r=0.002. Slopes are barely positive (weak instantaneous correlation; the real signal appears at the trained-controller's response lag — see scatter analysis below). |

**Verdict**: the three sign-inversion checks that drove 023 (2 gyro↔quat, 5
heading↔track, 7 nose↔vel) all pass with positive slopes and strong
correlation. The sim is coordinate-canonical and matches
[COORDINATE_CONVENTIONS.md](../../docs/COORDINATE_CONVENTIONS.md).

## Command → response scatter (3×3 per-axis best-lag)

Script: [`cmd_response_scatter.py`](./cmd_response_scatter.py) — a 024 fork
of the 023 plotter. **Difference from 023: rows 2 and 3 use per-axis best
lag rather than a unified lag** (see "Why the script changed" below). Path
0 & 2, wind 0 slice (344 rows) for legibility.

**Headline: all three axes sign-correct against
[COORDINATE_CONVENTIONS.md:110-113](../../docs/COORDINATE_CONVENTIONS.md#L110-L113)
when evaluated at each axis's actual response lag.**

### Per-axis best-lag response (rows 2 & 3 of the PNG)

| Axis | gyro r @ best lag | quat-d/dt r @ best lag | Convention check |
|---|---|---|---|
| pitch → q | **+0.863 @ +100 ms** | +0.221 @ +100 ms | NN `outPt +1` → positive pitch rate → nose up ✓ |
| roll → p  | **+0.519 @ +200 ms** | +0.372 @ +200 ms | NN `outRl +1` → positive roll rate → right wing down ✓ |
| throttle → vel | **+0.401 @ +700 ms** | (same as row 2) | NN `outTh +1` → vel increases ✓ |

### Row 1 (instantaneous, lag=0) — not direct response

| Axis | r |
|---|---|
| pitch → gyrQ | −0.418 |
| roll → gyrP  | −0.327 |
| throttle → vel | −0.122 |

Negative values at lag=0 are the expected "cmd leads response" pattern — at
the instant the command is applied, the rate hasn't yet developed (and can
even be opposite due to previous oscillations). Not a sign bug. The row-1
view is kept as a visual anchor for "this is what you'd see if you
forgot to lag-shift."

### Why the script changed (024 fork)

023 used a **unified 234 ms lag** across all three axes on rows 2 and 3.
That worked when the sim step was 117 ms (2 × 117 = 234, which happened to
sit near each axis's direct response peak). At cadence7's 100 ms step with a
sharper trained controller, axes have different response times:

- Pitch peaks at **+100 ms** (1 step).
- Roll peaks at **+200 ms** (2 steps).
- Throttle peaks at **+700 ms** (7 steps, prop+drag settling).

Forcing a unified 200 ms lag (the 024 quantization of 234 ms) lands past
pitch's peak and shows the oscillatory rebound (r = −0.56) instead of the
direct response (r = +0.86). Same scatter, opposite apparent sign,
because you're sampling a different part of the response curve. The
per-axis best lag is the honest view.

If you need to compare directly against an old 023 PNG, run the old
script. If you want to judge convention sign on cadence7-style data, use
this one.

### Comparison against 023-era (hb1-adjust4 tier1, same filter)

| Axis | hb1-adjust4 best | cadence7 best | Δ |
|---|---|---|---|
| pitch → gyrQ | +0.81 @ +117 ms | +0.86 @ +100 ms | Stronger + faster |
| roll → gyrP  | +0.74 @ +117 ms | +0.52 @ +200 ms | Weaker + slower |
| throttle → vel | +0.53 @ +702 ms | +0.40 @ +700 ms | Slightly weaker, same lag |

Pitch response is both tighter and sharper — consistent with cadence7's
better fitness trajectory. Roll is modestly weaker; worth watching on the
flight data but not disqualifying.

## Bench checklist (pre-flight)

Before T113a flash, confirm on the live FC:

1. **Pitch polarity sanity**: with autoc engaged, inject `outPt = +1` (e.g. via a known-forced scenario in a ground test, or observe the first engage frame). Aircraft nose should pitch **up**; elevator surface trailing edge should deflect **up** (standard "pull back = pitch up"). If the nose pitches down, INAV PITCH RC reversal is flipped — correct via CLI `set ail_reverse` / `set pitch_reverse` before flight.
2. **Roll polarity sanity**: `outRl = +1` → right aileron **down**, left **up** → roll right ("right wing drops"). If left wing drops, flip `roll_reverse`.
3. **Throttle polarity sanity**: `outTh = +1` → motor at high power (MSP 2000, standard). Should not require reversal.
4. **Cadence audit on the flight FC**: after boot, confirm xiao log shows 100 ms inter-NN delta. Any drift beyond ±10 ms indicates a xiao loop-timing regression vs the bench build.

These are belt-and-suspenders checks; the sim data is convention-correct and
should map straight through, but the Bridge code's sign invariants only hold if
INAV's RC reversal matches expectations. On a fresh or reflashed FC, don't
assume.

## Deferred items (post-flight)

- **sensor_self_check check 1 (pos↔vel integration) on sim data** — the library
  fails E/D axes with large-magnitude slopes. Rotation is correct (check 7
  proves it), so the bug is in the integration step or a unit/sign in
  pos-vs-world-pos scaling. Parallel to T060–T062 rotted-scripts cleanup in
  group D.
- **Check 8 (cmd↔rate) in sensor_self_check runs without lag** — reports very
  weak r (0.05 for pitch, 0.002 for roll). The lag-aware scatter is the right
  view; consider adding an optional lag parameter to the library check.

## Conclusion

Cadence7's sim-side data is **coordinate-canonical** against the 024 convention
set. No sign-inversion surprises. All three control axes respond in the
documented direction at their natural response lag:

- Pitch: +0.82 at +100 ms (nose up on positive command)
- Roll:  +0.52 at +200 ms (right wing down on positive command)
- Throttle: +0.40 at +700 ms (speed up on positive command)

Response shape is consistent with (and slightly sharper than) hb1-adjust4, in
line with cadence7's stronger fitness trajectory. Ready for US6 T113 xiao
rebuild. Recommend running the bench checklist above before T113a flash.

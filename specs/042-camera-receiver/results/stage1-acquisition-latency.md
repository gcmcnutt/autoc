# T074 — cold acquisition latency at the real bench operating point (2026-08-22)

**Conditions, all of which matter**: static scene, emitter and four ArUco sheets in the layout of the
2026-08-21 night session, **StepFPGA powered down**, **daytime background lighting**, exposure at the bench
operating point (53 µs, `beacon-bench.ini`), Pi 5, 288 fps / 120 Hz.

## Result

| run | first lock | present after lock | drop episodes |
|---|---|---|---|
| 1 | 2.25 s | 100 % | 0 |
| 2 | 6.95 s | 83 % | 1 |
| 3 | 4.15 s | 45 % | 3 |
| 4 | 2.10 s | 100 % | 0 |
| 5 | 2.95 s | 100 % | 0 |

**Cold acquisition: median 2.95 s, range 2.10–6.95 s, n = 5, 0/5 failures within 20 s.**
Against the §3 relock bar of 0.40 s that is **7× over at the median**, and the floor imposed by one code
word is 258 ms (§2.5.1) — so the gap is not the word, it is everything around it.

Once acquired the tracker is mostly stable at rest: 3/5 runs held 100 % with zero drops. Run 3 is the
outlier (45 %, three drop episodes) and shows the churn is not fully gone even with a static camera.

This supersedes the "~8 s" figure quoted in [stage1-pan2-analysis.md](stage1-pan2-analysis.md), which was
flagged there as *not banked* because it was taken at the fiducial exposure (1499 µs). It was right not to
bank it.

## ⚠️ The FPGA comparison is CONFOUNDED — do not draw it

The obvious temptation is to compare tonight's fast acquisition against last night's and credit the
StepFPGA's LEDs, which sit in the same field of view as the emitter. **Two things changed at once**: the
FPGA went off *and* the session moved from night to daytime, so the background illumination is different.
Operator flagged this directly (2026-08-22).

Daylight is known to matter here, not merely suspected — the bench journal's 2026-08-20 entry records that
*LED-PWM flicker out-blinks the beacon at short exposure*, and `acquire_pass`'s threshold is relative to the
field's mean |diff|, so the whole detection statistic moves with ambient. Attributing the difference to the
FPGA would be picking one of two changed variables.

**The clean A/B is cheap**: power the StepFPGA back on, same daylight, same layout, same five runs. That
holds everything else and isolates the LEDs. Until then the only defensible statement is the boxed result
above, with all four conditions attached.

# 039 Wrap — xiao 20 Hz flight

**Date**: 2026-07-13. Companion to [tasks.md](tasks.md) (T034–T038 record the bench-hardening day)
and `flight-results/flight-20260713/`. Verdict: **SUCCESS** — first 20 Hz autonomous flight,
**4/4 spans flown to `path complete`**, zero overruns / zero drops / zero tick gaps across 1339
engaged ticks, full field workflow (battery-only → BLE download → decode → blackbox-correlate)
proven end-to-end.

## 1. The arc

Bench latency memo → 20 Hz honest-servo enable → binary flight log v1 (bench) → v2 (self-contained
telemetry + breadcrumbs) → the 2026-07-12 waived-flight bench day (alignment HardFault root-cause,
fault_guard WDT+breadcrumb, v3 arm→disarm self-containment, blackbox correlation tool, flight-
hardware recovery) → flight 2026-07-13. The one flight-blocking bug of the feature — packed-struct
VLDR alignment fault masquerading as a "USB bus wedge" — was found on the bench the day before the
original flight window, which is exactly what bench days are for.

## 2. Flight 2026-07-13 (gen9200 M1, wind ~320@10 kt)

| span | path | ticks | end | track err mean/p95 (m) | Gz max/min |
|---|---|---|---|---|---|
| 1 | 0 StraightAndLevel | 341 | path complete | 12.4 / 20.2 | +8.29 / −6.90 |
| 2 | 2 | 439 | path complete | 20.9 / 35.3 | +8.17 / −5.58 |
| 3 | 3 | 136 | path complete | 17.1 / 22.4 | +6.61 / −5.27 |
| 4 | 1 SpiralClimb | 423 | path complete | 28.6 / 47.5 | +7.15 / −4.67 |

- **Loop health**: 0 overruns, 0 resyncs, worst lateness 26 ms, fetch/eval/send ≈ 13/0.9/5.5 ms,
  interval avg 50.0 ms — the 20 Hz claim holds in the air.
- **Instrumentation fidelity**: flight log vs blackbox across 4085 samples — pos err mean 16 cm,
  attitude mean 1.0°; the xiao's 141°/s engaged pitch-rate RMS matches blackbox's 140.5
  independently. v3 INAV_CLOCK anchors measured clock drift −902 ppm this flight vs +1513 on the
  bench → the xiao clock is RC-oscillator-class; **per-flight anchor fitting is required and works**
  (residuals ≤ 29 ms ≈ one fetch).
- **Tracking**: consistent trailing orbit, never diverged; straight-and-level ≪ spiral, consistent
  with wind on ground-fixed paths. No sim-eval comparison run yet (see §4).
- **Loads**: routine ±7–8 g in-span; flight max **+9.6 g happened OUTSIDE spans** (acro-mode
  recovery after disengage). Throttle 86% saturated at ⟨|u|⟩ 0.95 (the 037 un-lagged-actuator
  endpoint, plus wind) — 035-line energy objective material, nothing new.

## 3. The pitch finding — three control regimes, one marginal plant

Operator: this article is ~neutral CG (marginally stable in pitch), and the towed streamer is a
significant additional effect. During NN spans INAV runs `MANUAL|MSPRCOVERRIDE` — NN drives raw
surfaces, no inner loop. Blackbox, same flight, same airframe:

| regime | pitch-rate RMS | roll-rate RMS | elevator Δ/tick |
|---|---|---|---|
| INAV acro (untuned PID), launch | **24 °/s** | 40 | 2.0 |
| INAV acro, between spans | 53 | 60 | 6.1 |
| pilot MANUAL → landing | 50 | 43 | 7.0 |
| **NN direct (all spans)** | **141** | 93 | 71 |

The fast gyro inner loop makes the marginal plant "on a rail"; both slow outer loops (human, 20 Hz
NN) see it ring — the pilot manages 50 °/s with constant correction ("quite on edge"), the NN rides
at 141. The NN's pitch *commands* were the calm axis (⟨|u|⟩ 0.38, dCtrl 0.31 vs roll 0.70/0.46) —
the oscillation is closed-loop plant sensitivity, not command aggressiveness, which is why
command-domain per-axis reports (t5-style dCtrl) cannot see it. Static margin was never a craft
variation; every training rollout flew a self-damping sim plant. Holding 4/4 paths to completion on
an unpracticed marginal plant is a robustness result in itself.

## 4. Outcomes & decisions

1. **OPERATOR DECISION: no sim recalibration from this n=1 airframe.** Next flight articles are
   being built; wait for them before touching hb1_streamer pitch knobs (streamer effect alone is
   large). The quantified target is banked: reproduce ~141 °/s pitch RMS vs ~95 roll under the
   recorded command stream (`flight_report.py`, memory `project_model_convergence`).
2. **Candidate levers when articles arrive** (recorded, not chosen): forward CG ballast (cheapest,
   hardware); sim static-margin/pitch-damping match or a static-margin **craft-variation axis**
   (034-style, trains robustness across CG); architectural — NN over INAV's rate loop
   (acro+override instead of manual+override). The last changes the action space (setpoints, not
   surfaces) — sim must match; it's a feature of its own, not a tweak.
3. **v3 flight log + correlation tooling are the standing flight-analysis surface**:
   `flightlog_decode.py`, `correlate_blackbox.py`, `flight_report.py`. Every flight now
   self-describes program, disengage reasons, failsafe/switch history, and its own clock bridge.
4. **fault_guard is flight-proven infrastructure**: WDT 30 s (the-dog-survives-into-DFU discovery),
   .noinit crash breadcrumb. Field recovery recipe for a dead-firmware airframe (FC→DFU mode) is in
   memory `reference_xiao_usb_dfu_quirks`.
5. Deferred/backlog: sim-eval vs flight tracking comparison for this candidate (T027-class,
   fixed-eval per `project_late_run_fitness_interpretation`); energy/smoothness objectives for the
   ±8 g routine loads + pinned throttle; delta+varint@50 Hz already filed.

## 5. Handoff

- Flight data: `flight-results/flight-20260713/` (v3 bin, blackbox, decoded CSVs, plots)
- Next: new flight articles → repeat this flight protocol (it's now ~zero marginal analysis cost),
  then choose among §4.2 levers with n>1; 040 M2-depth work proceeds independently.

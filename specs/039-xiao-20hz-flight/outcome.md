# 039 Outcome Record

Running record of the operator-gated validation results (FR-002 bench, FR-011 soak,
latency memo decision, flight verdict). Bench artifacts under `eval-results/bench-*`.

## T010 — FR-002 observational bench span: **PASS** (2026-07-11)

**Artifact**: `eval-results/bench-20260712/flight_log_2026-07-12T01-17-40_flight_001.bin`
(11 KB, flight #1, span 1: 111 ticks @ 20 Hz, 5.5 s engaged, path 0 StraightAndLevel).
Decoded clean by BOTH readers (desktop `flightlog_decode.py` + in-browser
`flight_logger.html`) — identical span/summary numbers.
*(Note: this artifact is format **v1**; tooling moved to v2 — pos/vel/rabbit telemetry —
the same evening, pre-first-flight. Re-decoding it needs the v1 tooling at commit `94e5fde`;
the analysis above and the t1 plot are the durable record.)*

**Candidate identity** (banner ↔ FileHeader match): topology `37->32->16r->3`, 2051 w,
`weight_id=32fb43986aaaf3d1`, `firmware_id=d229e6045ad55de1`, arena template 80/5/100,
weights = `autoc-m1/autoc-9223370253553029228-2026-07-06T01:35:46.579Z/gen9200.dmp.zst`
(gen 800 t5 rebake elite).

**FR-002 checklist** (contracts/bench-validation.md):

| item | result |
|---|---|
| 1. boot banner topology + ids match candidate | ✓ (banner also printed at engage) |
| 2. EngageHeader re-centered arena | ✓ origin=(-11.1, 2.9, -3.3), floor_z=44.2 = z_e+47.5, ceiling_z=-50.8 = z_e−47.5 |
| 3. path moves around craft; reset marker | ✓ dist_now 0.6→58.9 m (rabbit 12 m/s), 107 distinct values; `recurrent_reset=1` exactly tick 0 |
| 4. all 37+3 plausible | ✓ no NaN/inf; unit fields in [-1,1]; quat norm err 0.0000; outputs alive (89/98/31 distinct); **center-of-arena readings at engage**: dist_to_boundary=1.000, inward_body=(0,0,0) — the documented on-axis case |
| 5. decoder clean, ticks contiguous | ✓ 0 gaps, 0 drops, 111/111 logged, state_valid all 1 |

**Cadence/pipeline (span summary)**: interval 45.0/50.0/55.0 ms, maxLate 4 ms,
avgLate 0.63 ms, **0 overruns / 0 resyncs**; fetch 6.8/13.1/34.3 ms,
eval 0.6/1.2/10.7 ms, send 3.9/5.4/8.9 ms, total 11.7/19.6/42.8 ms (min/avg/max).

## T019 — DWT eval cost (firmware d229e6045ad55de1)

**178858 cycles = 2.79 ms @64 MHz** — first eval after boot, bracketing
**gather + forward** (not forward alone). Steady-state eval segment averages 1.2 ms
(vs 2.6 ms in the 10 Hz era on the smaller 33/1923 network — unroll helped, but the
bracket is gather-dominated). Eval max 10.7 ms tail + first-eval inflation:
operator read = **interrupt-servicing variance** (nRF52840 has no meaningful
cache), not cache effects. Open before the memo: split gather vs forward-pass
brackets to attribute the cost (candidate firmware tweak pre-T020 soak).

**Visual**: [autoc-039-t1-bench-latency.png](autoc-039-t1-bench-latency.png) — tick-interval
series + distribution + per-section min/avg/max vs the 10 Hz era. Read: interval spread
(29–72 ms) is fetch jitter, not loop drift (maxLate 4 ms); fetch is the dominant,
unchanged, wire-clocked term; total max 42.8 ms vs the 50 ms tick is the margin story.

## Memo-relevant early deltas vs the 10 Hz era (R1 table)

| component | 10 Hz era mean/max | this bench mean/max |
|---|---|---|
| fetch | 12.6 / 31.1 ms | 13.1 / 34.3 ms (wire-clocked, unchanged) |
| eval (gather+fwd) | 2.6 / 7.4 ms | 1.2 / 10.7 ms |
| send | 8.9 / 16.2 ms | 5.4 / 8.9 ms |
| total | 24.2 / 43 ms | 19.6 / 42.8 ms |

Pending: T017/T020 soak (several consecutive spans), T021 baud experiment,
T022 memo, T023 decision, T025-T028 flight.

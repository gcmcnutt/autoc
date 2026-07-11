# 039 Research — latency ground truth, log completeness/compression, regen/transport surface

**Date**: 2026-07-10 (plan phase; the spec's two primary research items ran here per operator direction).
Sources: flight-results/flight-20260503 + flight-20260517 logs, crrcsim + xiao + INAV-fork code,
037 contracts. All numbers cite files inspected read-only.

## R1 — Latency: measured real pipeline vs crrcsim model (US2 / FR-004..006)

### Measured (real, 10 Hz-era firmware, two independent flights agree)

Per-span MSP pipeline stats printed by the xiao (`xiao/src/msplink.cpp:146-151`; e.g.
`flight-results/flight-20260517/flight_log_2026-05-17T19-55-07.txt:592,1202`):

| component | mean | 95%ile | max |
|---|---|---|---|
| fetch (MSP2_AUTOC_STATE round trip) | 12.6 ms | ~12.8 ms | 23.9–31.1 ms |
| NN eval (table-driven, 33-in/1923-w) | 2.6 ms | ~2.6 ms | 6.5–7.4 ms |
| send (MSP_SET_RAW_RC) | 8.9 ms | ~9.2 ms | 12.2–16.2 ms |
| **total** | **24.2 ms** | — | **36–43 ms** |

Tick cadence: 100.0 ms mean, 95.6–104.4 ms spread, zero overruns — and the firmware **already
carries loop-health counters** (`ctl loop: ticks/overruns/resyncs/maxLate/avgLate` per span,
`xiao/src/msplink.cpp:163-168`); 039 keeps them and moves them into the binary log (console is
demoted per FR-014). Downstream of the xiao: INAV RC application ≈ 5 ms (estimate, not directly
logged); servo ≈ 82.5 ms full-throw (DSM-44, 55 ms/60°), rcData→gyro-response correlation lag
10–12 ms (`flight-results/flight-20260517/FLIGHT_REPORT.md:174-182`).
**INAV RC filtering is intentionally not a factor** (operator: "we did a lot to set that to 0 vs
modelling it") — config shows `rc_filter_lpf_hz = 250`, `rc_filter_auto = OFF`
(`xiao/inav-hb1.cfg:1401-1403`): a 250 Hz cutoff is transparent (sub-ms group delay) at our command
rates. The memo must NOT model or chase RC-filter lag.

### Modeled (crrcsim)

- **Compute latency**: command-delay queue, `COMPUTE_LATENCY_MSEC_DEFAULT = 30` ms
  (`crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h:56`; env override
  `AUTOC_COMPUTE_LATENCY_MSEC`, applied `inputdev_autoc.cpp:393-401`).
- **Servo v2** (`crrcsim/src/mod_fdm/fdm_larcsim/fdm_larcsim.cpp:248-310`): 50 Hz PWM latch with
  per-scenario 0–20 ms phase + slew ~24.2 u/s (≈82.5 ms full throw) — **matches the measured
  hardware closely** (037 finding.md:255-280).

### Gap analysis

| | real | modeled | verdict |
|---|---|---|---|
| xiao pipeline + INAV apply | 24.2 + ~5 ≈ **29 ms** mean | **30 ms** | ✓ remarkably close |
| fetch/send jitter (6.8–31 ms fetch tail) | present | absent (deterministic delay) | gap — jitter unmodeled |
| tick-period jitter (±4.4 ms) | present | absent | gap (BACKLOG "Sampling Time Variation") |
| servo latch + slew | 0–20 ms + 24.2 u/s | same | ✓ modeled |

**Decision (research recommendation — operator decides at review per clarification #4):** the sim's
30 ms mean is honest for the 10 Hz-era pipeline (within ~1 ms of measured mean+INAV). The
**mean does not justify a retrain by itself.** Two things change the picture at 20 Hz and must be
re-benched on the regenerated firmware before the memo is final: (a) unroll cuts eval 2.6 → ~0.1 ms;
(b) a baud raise cuts wire time (fetch mean is mostly clocking the 67-byte frame at 115200 ≈ 6.4 ms).
A 20 Hz pipeline at, say, ~12–15 ms mean would make the modeled 30 ms **conservative by 2×** — that
gap (not today's) is the retrain question. The worst-case 43 ms total in a 50 ms tick is the
operational risk the bench must characterize (tail, not mean).
**Alternatives considered**: retrain now on measured 24 ms (rejected: the number is stale the moment
the 20 Hz firmware lands); model fetch/send jitter (deferred to BACKLOG jitter-dither item — it's a
robustness lever, not a mean-latency fix).

### Holes needing the fresh bench (FR-004)

Sub-phase breakdown of the fetch tail (serial RX vs INAV reply jitter vs retry), true INAV
MSP-to-rcData apply time, isolated servo step response, and the 20 Hz numbers post-unroll/baud.

## R2 — Flight log: completeness audit + encoding choice (US3 / FR-007..009, FR-014)

### Current format (029-vintage)

Text lines (`xiao/src/msplink.cpp:310-336`): per-tick `NN:` line avg **327 B** (33 inputs era) plus
`Nav State:` line ~268 B. Flash: 2 MB QSPI region minus 4 KB metadata; 3×4 KB RAM buffers, 256 B page
writes, async flush; **no in-loop data erase** (pre-erased ring; ground `ERASE:ALL`). Capacity math:
**10.7 min @ 10 Hz text; 5.3 min @ 20 Hz text — text blocks 20 Hz** (the 023 prediction confirmed
with real line sizes from `flight-results/flight-20260503/flight_log_2026-05-03T18-13-21.txt`).

### Completeness gaps vs the 038 contract (37 in + 3 out + aux)

Missing today: `dist_to_boundary`, `inward_body[3]` (the 4 new 038 inputs), per-tick timestamp,
span/engage id, recurrent-reset (warm-up) marker, **arena origin per engage** (new FR-001), RC-sent
values, state-valid flag.

### Target record (drives data-model.md)

- **Engage header** (once per span): engage timestamp, arena origin NED[3] + K, span id ≈ 22 B.
- **Per-tick record**: timestamp + counter + 37 inputs + 3 outputs + reset flag + path idx
  (+ optional RC/valid telemetry) ≈ **169–176 B raw**.

### Encoding candidates (per-tick, 20 Hz, 8-min budget = 2×4 min engaged)

| encoding | B/tick | 8-min total | fits 2 MB? | RAM | decode |
|---|---|---|---|---|---|
| text (current) | 327+ | 2.99 MB | ✗ | — | trivial |
| raw binary float | 179 | 1.64 MB | ✓ | ~0 | trivial |
| **int16 quantized packed** | **93** | **0.85 MB** | **✓ (58% headroom)** | ~0 | scale tables |
| delta+varint keyframed | ~65 | 0.60 MB | ✓ | ~200 B | stateful |
| heatshrink LZSS on frames | ~89 | 0.81 MB | ✓ | 50–100 B | streaming |

Write bandwidth is a non-issue for all binary candidates (≤3.5 KB/s vs QSPI page-write path);
the constraint is **capacity**, and after that **BLE download time** (0.85 MB ≈ 7 s vs text 24 s).

**Decision: int16-quantized packed binary, versioned (v1).** NN values are tanh/unit-bounded —
int16 on [-1,1] gives ~3×10⁻⁵ resolution, far below actuation resolution; distances/positions get
per-field scale tables. Simplest encoder (multiply+saturate), stateless (a corrupted page loses one
tick, not a stream), fits the budget with 58% headroom, and 3× faster BLE turnaround.
**Alternatives**: delta+varint (best ratio, but stateful decode + corruption blast radius —
**deferred to the 50 Hz era**, which is exactly where 037's packed-log contract aimed); heatshrink
(adds a dependency for a worse ratio than quantization here); raw float binary (fallback if
quantization QA finds a field that can't scale — costs the headroom).
**Console split (FR-014)**: with the binary log carrying everything, console shrinks to events +
~2 Hz heartbeat — this also removes the per-tick `Nav State:`/`NN:` text formatting cost
(~600 B of snprintf per tick) from the loop.

## R3 — Firmware regen surface + transport budget (US1 / FR-001..003, US4 / FR-011)

### Regen change set

- Current generated file is 029-vintage: `33→32→16r→3`, 1923 w, old `gather_pathgen_inputs`
  signature (`xiao/src/generated/nn_program_generated.cpp:1,284`).
- `tools/nn2cpp.cc` already emits the 038 contract: `-a R,F,C` baked-arena flag (lines 43-56),
  gather call with arena param; xiao build already pulls desktop `src/nn/evaluator.cc` +
  `src/eval/sensor_math.cc` (`xiao/platformio.ini:31-34`); `arena.h` is cereal-free/xiao-safe.
- Weight source: pinned t5 elite `autoc-m1/autoc-9223370253844606963-2026-07-02T16:36:08.844Z/`
  gen 800 (37,32,16r,3 → 2051 w).
- **Gap 1 — engage-time arena**: nn2cpp bakes a *static const* FlightArena; FR-001 requires
  re-centering per engage (`floor_Z = z_engage+K`, `ceiling_Z = z_engage−K` — pure ±K, no
  min-elevation clamp per the 2026-07-10 simplification,
  K = (ceiling−floor)/2 = **47.5 m** for the 80/5/100 training arena). **Decision**: the firmware
  holds a *mutable* engage-scoped FlightArena (recomputed in the span-activation path from
  `z_engage`), passed to `gather_pathgen_inputs` — nn2cpp's baked literal becomes the geometry
  *template*, not the placement. The template is passed as `-a R,F,C` (80,5,100 — same flag surface
  as today); the firmware derives K = (C−F)/2 = **47.5 m** and applies the engage-time rule.
  Alternatives: keep static arena (violates FR-001); change the gather signature (unnecessary — it
  already takes `const FlightArena&`).
- **Gap 2 — unroll doesn't cover recurrent**: `-u` falls back to table-driven when the genome has a
  recurrent layer (`tools/nn2cpp.cc:356-361`) — and the elite is 16r. **Decision (operator confirmed
  2026-07-10)**: implement unrolled-recurrent emission in nn2cpp — the recurrent part unrolls fine
  with **persistent intermediate state registers**: a static hidden-state array `h[16]` surviving
  between calls (reset by `nn_reset()` at engage), each tick computing straight-line
  `h_new[j] = tanh(Σ W_xh·x + Σ W_hh·h + b)` into a temp buffer then committing (double-buffered to
  avoid read-after-write on `h`), exactly the table-driven semantics with the loops flattened.
  Desktop unit test asserts unrolled output ≡ table-driven output on the same weights (same
  accumulation order → bit-comparable on desktop). Expected on-target gain ≈ 2.6 ms → ~0.1 ms class
  (037 eval-cycle-harness contract has the DWT cycle-count harness design).

### Transport budget (20 Hz)

Per tick: `MSP2_AUTOC_STATE` 67 B frame (58 B payload: `xiao/include/MSP.h:262-272`) +
`MSP_SET_RAW_RC` 25 B = **92 B/tick vs 523 B available at 115200 baud** (11-bit chars) — **17.6%
utilization; 20 Hz fits at current baud, no format redesign needed for bandwidth**. The custom
query/response redesign lever stays parked unless the latency bench wants it: wire time is the
latency lever (67 B ≈ 6.4 ms at 115200 → 1.6 ms at 460800), so a **baud raise is the cheap
latency win**, evaluated in the FR-004 bench. `MSP_NN_EVAL_DIVISOR` currently 2 (10 Hz NN on a
20 Hz MSP loop, `xiao/include/main.h:23-26`) → 1.
INAV side: on-demand state serve (xiao drives cadence; `inav/src/main/fc/fc_msp.c:673-712`
consolidated handler), standard RC pipeline + ~2 s LOS failsafe — FR-013's safety envelope is the
existing one.

## Consolidated decisions

| # | decision | drives |
|---|---|---|
| D1 | Sim latency model likely stands on today's mean (30 vs ~29 ms); the retrain question is re-opened by the *20 Hz bench* numbers (post-unroll, post-baud) — memo + operator review at implement time | US2, FR-004/005 |
| D2 | Local IMU stays deferred: state-fetch round trip (≈12.6 ms mean, improvable by baud) fits the 50 ms tick; no fast-loop IMU needed for 20 Hz | FR-006 |
| D3 | Flight log v1 = **int16-quantized packed binary, versioned header**, engage header + per-tick record carrying all 37+3+aux; delta/varint deferred to 50 Hz era | US3, FR-007/008/009 |
| D4 | Console split lands with the format change (events + 2 Hz heartbeat; zero per-tick console) | FR-014 |
| D5 | Firmware holds a mutable engage-centered arena (template geometry from nn2cpp `-a`); K = 47.5 m | FR-001 |
| D6 | nn2cpp gains unrolled-recurrent emission + desktop equivalence test; DWT harness measures on-target | FR-003 |
| D7 | Transport format redesign NOT needed at 20 Hz (82% margin); baud raise is an optional latency lever decided by the bench | US4, FR-011 |

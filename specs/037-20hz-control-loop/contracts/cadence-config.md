# Contract: Cadence Configuration & Tick-Rescale

**Scope**: Phase A. The control-rate knob and the invariant that every cadence-coupled quantity stays
correct when it changes.

## Config surface

- **Key**: control interval (ms), read from `.ini`/CLI (R6) — NOT the `AUTOC_EVAL_INTERVAL_MSEC` env var.
- **Default 100, like every other knob** (operator decision 2026-06-09): a missing key is NOT a fail-loud.
  The value is auto-printed at startup (`AUTOC_CONFIG_FIELDS`), so a defaulted cadence is auditable in the
  log, not a *silent* fallback — which was the Principle-VII concern. (Erroring on a missing key would be
  an outlier in a config where everything else defaults.)
- **Single source of truth / coherence**: `ControlIntervalMsec` MUST equal `SIM_TIME_STEP_MSEC` (autoc),
  and `gEvalUpdateIntervalMsec` (crrcsim) derives from it via `WorkerInit`. config.cc **fails loud on
  `!= SIM_TIME_STEP_MSEC`** — that catches changing the cadence without the compiled constant (the real
  invariant). `SIM_TIME_STEP_MSEC` stays the master (it sizes compile-time arrays/static_asserts).

## Invariants (MUST hold)

1. `controlIntervalMsec % cycleLengthMs == 0` where `cycleLengthMs = round(1000/outerFps/fdmDtMs)*fdmDtMs`
   (already enforced — `inputdev_autoc.cpp:297-316`; keep the fatal-exit on violation).
2. `framesPerEval = controlIntervalMsec / cycleLengthMs ≥ 1`.
3. FDM oversamples the control rate ≥10×: `(1/fdmDt) / (1000/controlIntervalMsec) ≥ 10`. The 50 Hz arm
   needs `fdmDt ≤ 0.002` (≥500 Hz); 2 kHz (matching INAV `looptime=500`) satisfies all arms.

## Tick-rescale contract (MUST verify per term)

Any quantity denominated in ticks MUST produce the same *physical* meaning at the new rate:

| Term | Contract |
|---|---|
| per-tick accumulators (stability, energy) | normalized to per-second (Σ·dt) OR rescaled so total ≈ rate-invariant |
| rate derivatives (closing/closure) | divide by the **actual dt**, never a hardcoded `0.1f` |
| streak ramp | derived from seconds (already ✅) |
| engage delay | `ticks = ceil(EngageDelayMs / controlIntervalMsec)` — duration rate-independent |
| history depth | time-based (see nn-input-layout.md), not fixed ticks |

## Tests (Principle I)

- Unit: stability/energy total for a fixed synthetic trajectory is ~equal at 10 Hz vs 20 Hz (within FP).
- Unit: closing_rate for a known dDist is correct at two intervals.
- Unit: engage-delay tick count = ceil(ms/interval) at 10/20/50 Hz.
- Startup: cadence-triple assertion fires on a non-integral config.
- Regression: FP-deterministic bit-replay still matches at the unchanged 10 Hz config (operator-driven).

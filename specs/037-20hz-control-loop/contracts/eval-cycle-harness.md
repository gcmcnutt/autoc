# Contract: NN-Eval Cycle-Count Harness (R4)

**Scope**: Phase 0 research (parallel to A). A small program to MEASURE max-throughput unrolled NN eval —
not trust the ~0.1 ms theoretical floor. Cheap; only a Xiao on the bench.

## Two levels (both required)

### On-target (ground truth)
- A minimal nRF52840 sketch that runs the **codegen'd unrolled M1/M2 forward** in a tight loop and counts
  cycles with the **Cortex-M4 DWT cycle counter** (`DWT->CYCCNT`; enable `CoreDebug->DEMCR |=
  TRCENA`, `DWT->CTRL |= CYCCNTENA`). Today's firmware times with `micros()` — the harness adds DWT for
  cycle-accurate attribution.
- Report cycles + µs@64 MHz separately for: (a) MACs only, (b) +tanh (naive `expf` vs poly/LUT
  fast-tanh), (c) +input-gather (dir-cosines/dist/quat sensor math).
- Run for both shapes: M1 (33→32→16→3, 1923 w) and M2 (54→32→16→3, 2595 w).

### Off-target (fast iteration)
- The same codegen'd C compiled on host as an op-counter (count FMAs + transcendentals, or run under an
  M4 instruction-cost model) to iterate layout/unroll variants without a flash cycle. Confirm the winner
  on-target.

## Requirements (MUST)

1. **fp32** (sim↔real parity — no fixed-point).
2. Measure the **real** core (cache/flash-wait behavior), not a paper FLOP count.
3. Output a table `{shape × tanh-impl × phase} → cycles, µs` → the **defended eval budget** for the
   chosen control rate and the **Phase-C go/no-go** (is unroll even needed?).
4. Self-contained / does not perturb the flight firmware; lives under `xiao/` as a bench target.

## Acceptance

- A reproducible cycles/µs table for M1 and M2; a one-line recommendation: "unroll needed at rate X:
  yes/no" with the budget math.

# Contract: INAV ↔ Xiao Transport Link

**Scope**: Phase B. The read+write budget and the rate it must sustain.

## Current (measured / configured)

- UART `Serial1.begin(115200)` (`xiao msplink.cpp:342`); INAV side all `serial` ports 115200
  (`inav-hb1.cfg`). STM32F722 FC, `looptime=500` → 2 kHz inner loop.
- **Read** `MSP2_AUTOC_STATE` (pos/vel/quat/gyro/flags) — 12.6 ms avg / 25.6 ms max
  (`msplink.cpp:364`).
- **Write** `MSP_SET_RAW_RC` (roll/pitch/throttle + CH6 manual) — 9.2 ms avg / 12.2 ms max
  (`msplink.cpp:669-678`). Already sent every 50 ms (20 Hz) with duplicate contents
  (`MSP_NN_EVAL_DIVISOR=2`).

## Requirements (MUST)

1. **Account for both directions** in the per-tick budget: `read + write ≤ tick − (eval + fusion + log)`.
2. **Raise baud first** (cheapest): target 921600/1M; verify nRF52840 UARTE max stable rate and INAV MSP
   baud override on the actual harness/length. Re-measure read/write at the new baud.
3. If baud-bumped UART can't sustain the chosen rate's read+write, **escalate to SPI** (nRF52840 SPIM
   ~32 MHz / STM32F7 SPI) with a master/slave framing — documented as a separate decision.
4. **Per-tier poll rates** (Phase B): fast (local IMU, no MSP), intermediate (INAV position poll),
   slow (INAV attitude resync). Each tier's MSP cost must fit the link budget at its cadence.
5. **Latency compensation**: forward-propagate INAV's stale synced attitude to "now" with local gyro via
   the MSP `timestamp_us` before blending — don't blend the stale value directly.
6. Command frame must keep CH6 = manual (no INAV stabilization) — preserve current semantics.

## Acceptance

- Bench: read+write round-trip at the bumped baud ≤ the budget for the chosen rate, with margin for
  eval+fusion+log within one tick (no `ctl loop` overruns in `LoopStats`).
- Document the chosen transport + the resulting budget in the outcome doc; update `project_sim_latency`.

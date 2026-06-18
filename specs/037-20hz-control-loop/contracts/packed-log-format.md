# Contract: Packed Dual-Stream Flight Log (P5, Phase B — deferred)

**Scope**: Phase B prework, **deferred until after the Phase-A M1 signal** (operator, 2026-06-09).
Replaces the verbose ~328-char text dump so high-rate dual-stream (local IMU + INAV) capture fits flash.

## Requirements (MUST)

1. **Variable frame type** per rate tier: fast IMU frame, intermediate position/nav frame, slow
   attitude-resync frame, and a cross-check frame pairing local-IMU vs INAV.
2. **Variable frame rate**: each type emitted at its own tier cadence, not one fat per-tick line.
3. **Compact/differential encoding**: delta vs previous frame of that type + varint/bitpack; periodic
   keyframes for resync.
4. **Single authoritative writer/reader pair** (`project_log_format_shared_parser`); `join_flight_
   analysis.py` MUST decode it.
5. **Tail-safe write path**: pre-erased ring, async-DMA, **no in-loop erase** — fixes the blocking
   `qspiEraseBlocking`/`qspiWriteBlocking`/`saveMetadataBlocking` tail (`flash_logger.cpp:666-713`,
   the prime 7.4 ms-max suspect when armed). Under overrun: drop/coalesce the log, never stall control.

## Testing methodology (the validation flights this enables)

`activate → capture → confirm`: bring the local-IMU path up **first as a logged cross-check beside the
live INAV path** (don't hand it control), fly, confirm local fusion agrees with INAV
(attitude/heading/convention) within tolerance → then promote to driving the fast loop. The 021
convention cross-check is the gate.

## Acceptance

- Sustains the chosen rate × two streams within the QSPI write budget with no control-tick overrun.
- Round-trip decode in the shared reader + `join_flight_analysis.py`.

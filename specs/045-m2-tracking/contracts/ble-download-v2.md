# Contract: BLE flight-log download, v2

Service `F1706000-…` (unchanged UUIDs): control `…6001` (write), data `…6002` (notify), status `…6003` (notify).

## Step 1 — instrumentation (ships first, protocol unchanged)

- Firmware status lines: `MTU:<att_mtu>` at connect; `STAT:<offset>:<short_writes>:<max_loop_ms>:<max_next_gap_ms>` every 16
  chunks and in `DOWNLOAD_DONE`.
- Page log (in memory, **Copy log** button): READY time; per chunk: send time, receive time, `byteLength` vs expected, offset;
  errors with offset; disconnect offset and elapsed time; summary of bytes, seconds, kB/s, CRC result.

## Step 2 — protocol v2

| page → firmware (control) | firmware → page |
|---|---|
| `DL2:<file>:<offset>:<window>` | status `READY2:<size>:<chunk>:<mtu>` (chunk = MTU − 3 − 4) |
| — | data notifications, each `[u32 offset][payload]`, sent back-to-back up to `window` chunks unacknowledged |
| `ACK:<offset>` (the highest contiguous byte received) | the window slides; resend from `offset` on a gap |
| `ABORT` | stop, clear transfer state |
| — | status `DONE2:<size>:<crc32>` at the end; `ERR:<reason>:<offset>` on failure |

Rules:
- The firmware **requests MTU 247** after connect. If a notify's returned length is < chunk + 4 it **stops and reports
  `ERR:SHORT_NOTIFY`** (fail loud; never truncate silently).
- **Resume**: a new `DL2` with the next needed offset continues; the page keeps the received buffer across reconnects, keyed by
  file name + size.
- **Inactivity timeout**: no ACK for 5 s ⇒ the firmware clears transfer state and returns flash to idle (fixes the wedge at
  `flash_logger.cpp:491`).
- While a download runs **on the ground (disarmed)**, `controllerUpdate` skips MSP polling. Arming still stops BLE as today.
- Whole-file CRC32 is still checked at the end.

Acceptance: spec SC-002 (~300 kB < 60 s at field distance; an interrupted download resumes and is byte-identical to a wired read).
